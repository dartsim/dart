Examples
========

For interactive worlds, Linux display-free PNG capture, and the DART 7 Filament
renderer, see :doc:`the visualization guide <../../dart/user_guide/visualization>`.
The ``dartpy.gui`` classes retain the world-node simulation callback names;
they are interim bindings that DART 7's Python rendering API replaces before
release.

Once you have installed dartpy using pip install -U dartpy, you can run the following "Hello World" example to simulate a 6-DOF robot using DART.

.. note::

   In order to load the URDF, please clone dart repository and set the `DART_DATA_LOCAL_PATH` environment variable to where the `data` folder is in the cloned repository (e.g., `C:/ws/dart/data/` if cloned to `C:/ws/dart/`)

.. code-block:: python

    import dartpy as dart

    def main():
        world = dart.simulation.World()

        urdf_parser = dart.utils.DartLoader()
        arm = urdf_parser.parseSkeleton("dart://sample/urdf/six_axis_arm/six_axis_arm.urdf")
        ground = urdf_parser.parseSkeleton("dart://sample/urdf/six_axis_arm/ground.urdf")
        world.addSkeleton(arm)
        world.addSkeleton(ground)
        print("Robot {} is loaded".format(arm.getName()))

        for i in range(100):
            if i % 10 == 0:
                print(
                    "[{}] joint position: {}".format(
                        world.getSimFrames(), arm.getPositions()
                    )
                )
            world.step()

    if __name__ == "__main__":
        main()

The arm starts at zero joint angles in an upright L, with a vertical upper arm and a horizontal forearm. When you run this script, it will perform a forward dynamic simulation of the 6-DOF robot for 100 steps. Joint angles are printed every 10 steps, producing the following output:

.. code-block:: none

   Robot six_axis_arm is loaded
   [0] joint position: [0. 0. 0. 0. 0. 0.]
   [10] joint position: [ 0.00000000e+00  1.56990893e-06  2.44957094e-03  0.00000000e+00
    -5.26233357e-03  0.00000000e+00]
   [20] joint position: [ 0.00000000e+00  2.45210566e-05  9.33435506e-03  0.00000000e+00
    -2.00914590e-02  0.00000000e+00]
   [30] joint position: [ 0.          0.00012299  0.02059999  0.         -0.04447795  0.        ]
   [40] joint position: [ 0.          0.00038681  0.03615581  0.         -0.07838581  0.        ]
   [50] joint position: [ 0.          0.00094134  0.05587419  0.         -0.12172263  0.        ]
   [60] joint position: [ 0.          0.00194683  0.07959015  0.         -0.17429913  0.        ]
   [70] joint position: [ 0.          0.0035972   0.10710134  0.         -0.23578037  0.        ]
   [80] joint position: [ 0.          0.00611769  0.13816923  0.         -0.30563343  0.        ]
   [90] joint position: [ 0.          0.00976148  0.17252223  0.         -0.38307829  0.        ]

You can find additional example code at https://github.com/dartsim/dart/tree/main/python/examples

Interactive demo catalog
------------------------

The Python demo catalog runs on the native app host through
``dartpy.viewer.run()``. The host provides a categorized navigator,
simulation toolbar, dockable scene panels, and diagnostics.
Scenes supply worlds, callbacks, and declarative panels; Python does not draw
the UI. All former C++ scenes are available as Python scenes, except the
retired ``human_joint_limits`` scene.
The C++ demo application and its alias executables are removed. Open the
soft-body scenes with ``pixi run demos -- --scene soft_bodies``,
``--scene soft_cubes``, or ``--scene soft_open_chain``.

From a DART checkout:

.. code-block:: shell

   pixi run demos
   pixi run demos -- --scene rigid_cubes
   pixi run demos -- --list-scenes
   pixi run cycle-demos
   pixi run capture rigid_cubes
   pixi run demos -- --scene rigid_cubes --headless --shot cubes.png

Listing and cycling scenes require no graphics device. The app host requires
the bundled docking ImGui; a build using vanilla system ImGui omits
``dartpy.viewer``. Rebuild with ``-DDART_USE_SYSTEM_IMGUI=OFF``;
``pixi run config`` passes this option.

Versioned humanoid models
-------------------------

The optional ``utils-assets`` component provides
``dart.utils.ModelResourceRetriever`` for Atlas v5 and Unitree G1. Pixi source
builds enable it. Native builds require ``-DDART_BUILD_UTILS_ASSETS=ON``,
libcurl 7.85 or newer, and OpenSSL Crypto 1.1.1 or newer; published wheels may
omit this optional component.

From a DART checkout, download the pinned catalog once and open a model demo:

.. code-block:: shell

   pixi run fetch-robot-assets
   pixi run demos -- --scene unitree_g1
   pixi run demos -- --scene atlas_v5
   pixi run test-robot-assets -- --offline

The demos inspect joint poses with a fixed base. Atlas v5 is the historical
hydraulic robot without its head. These examples do not supply walking
controllers or establish hardware fidelity.

The humanoid scenes are registered only when ``ModelResourceRetriever`` is
available. They use the verified cache offline and report missing assets as
skipped scenes, so a scene cycle still succeeds before the models are prefetched.
The Python scene panels provide joint and position sliders to change joint
angles within the model limits, plus a reset button.

Applications can explicitly retrieve models online on first use:

.. code-block:: python

   import dartpy as dart

   models = dart.utils.ModelResourceRetriever()
   uri = (
       "model://unitree-g1/"
       "5994d4faef0a9cadd3287f8de0199a67eeb2a259/"
       "g1_29dof_mode_15.urdf"
   )
   options = dart.utils.DartLoader.Options(
       models, dart.utils.DartLoader.RootJointType.FIXED
   )
   loader = dart.utils.DartLoader()
   loader.setOptions(options)
   robot = loader.parseSkeleton(uri)
   if robot is None:
       raise RuntimeError("Model retrieval or parsing failed; see DART diagnostics")
   print(robot.getNumDofs())  # 29

The default cache lives under the platform's user-cache directory. Pass
``cacheDirectory="/chosen/cache"`` to select a location or ``offline=True``
to forbid downloads. The complete model directory is fetched and verified
against the installed manifest's SHA-256 hashes and sizes before use. Revisions
coexist in separate directories; downloaded assets are outside DART's
installed sample-data tree.

``models.addManifest("/path/to/model.xml")`` registers a caller-supplied local
manifest. It must enumerate all referenced model, mesh, texture, and metadata
files with pinned HTTPS URLs, checksums, and byte counts. The schema and full
examples live in ``data/robot_models``. For URDF package references, compose a
``PackageResourceRetriever`` with this retriever and map package roots to the
corresponding versioned ``model://`` directory.

Missing offline files and corrupt caches fail with a diagnostic. Remove only
the reported corrupt bundle directory and prefetch it again. Keep the cache
unchanged while a scene is loaded, because the renderer reads some textures
directly from local paths.

Retrieval and model parsing release Python's GIL. Synchronize changes to loader
options or package mappings while a retrieval or parse call is active.

The historical ``hubo_puppet`` and ``fetch`` demo selectors have been retired.
Hubo's installed ``dart://sample/urdf/drchubo/`` assets were removed. Fetch and
the older Atlas models used by parser and controller examples remain available.

Analytical inverse kinematics with ssik
---------------------------------------

dartpy can plug an external Python analytical IK solver into DART's native
``InverseKinematics::Analytical`` pipeline through
``InverseKinematics.setPythonAnalytical``. The binding is solver-agnostic: you
supply any callable that maps a desired end-effector transform (a 4x4 NumPy
matrix) to a list of joint-space solutions, and DART keeps its native solution
selection, joint-limit handling, and extra-DOF utilization.

The `ssik <https://github.com/personalrobotics/ssik>`_ package provides
closed-form analytical IK for many common arms and is a convenient solver to
plug in. ssik is an optional, PyPI-only dependency; it is not required by DART or
dartpy and is only used by the IK examples:

.. code-block:: shell

   # from a DART checkout
   pixi run install-ssik
   # or directly
   pip install "ssik>=8.1.0"

The sketch below registers an ssik prebuilt solver on an end effector and solves
for a target pose. Build the skeleton so its DOFs match the ssik chain with the
base at the world origin, so the world-frame target equals the frame ssik
expects:

.. code-block:: python

    import dartpy as dart
    import numpy as np
    from ssik.prebuilt import ur5_ik as arm

    # ... build `skeleton` from the ssik chain and obtain the tip body `ee` ...

    ik = ee.getOrCreateIK()
    dofs = list(range(skeleton.getNumDofs()))
    ik.setDofs(dofs)

    def solve(target):  # target: 4x4 NumPy matrix in the base frame
        return list(
            arm.solve(np.asarray(target).reshape(4, 4),
                      max_solutions=None, respect_limits=True)
        )

    analytical = ik.setPythonAnalytical(solve, dofs, "ssik.prebuilt.ur5_ik")

    # Solve for a desired end-effector pose and apply the best solution.
    target = dart.math.Isometry3()
    # ... target.set_rotation(...) / target.set_translation(...) ...
    solutions = analytical.getSolutions(target)
    if solutions:
        skeleton.setPositions(np.asarray(solutions[0].mConfig))

A complete demo is the ``ssik_ik_gui`` scene in the Python demo catalog,
implemented in ``python/examples/demos/scenes/ssik_ik_gui.py``. It builds native
DART skeletons from the solver's product-of-exponentials chain and offers 19
arms, position and roll/pitch/yaw controls, a draggable target, solver options,
seed selection, solution diagnostics, and up to 16 solution skeletons.
Select **Read dragged** to copy the dragged target into the controls, then
**Solve** to solve. **Seed from selection** seeds the next solve from the
selected solution. Install the optional solver before running:

.. code-block:: shell

   pixi run install-ssik
   pixi run demos -- --scene ssik_ik_gui
   pixi run dartpy -m pytest python/tests/integration/test_demo_ssik.py -q

The tests check forward kinematics and DART's Python analytical IK integration
over all 19 arms. When ``ssik`` is unavailable, scene cycles report this scene
as skipped.
