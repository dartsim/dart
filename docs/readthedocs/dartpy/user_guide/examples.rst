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
        kr5 = urdf_parser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
        ground = urdf_parser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
        world.addSkeleton(kr5)
        world.addSkeleton(ground)
        print("Robot {} is loaded".format(kr5.getName()))

        for i in range(100):
            if i % 10 == 0:
                print(
                    "[{}] joint position: {}".format(
                        world.getSimFrames(), kr5.getPositions()
                    )
                )
            world.step()

    if __name__ == "__main__":
        main()

When you run this script, it will perform a forward dynamic simulation of the 6-DOF robot for 100 steps. Joint angles are printed every 10 steps, producing the following output:

.. code-block:: none

   Robot KR5sixxR650WP_description is loaded
   [0] joint position: [0. 0. 0. 0. 0. 0.]
   [10] joint position: [ 0.00220342  0.00021945 -0.00040518  0.00011133  0.00074889 -0.00010902]
   [20] joint position: [ 0.00841056  0.0008539  -0.0015611   0.00042308  0.00284968 -0.00040791]
   [30] joint position: [ 0.01861372  0.00194988 -0.00350848  0.00092958  0.0062733  -0.00087254]
   [40] joint position: [ 0.03279843  0.00358421 -0.00631463  0.00162151  0.01097006 -0.0014636 ]
   [50] joint position: [ 0.05094093  0.00586373 -0.01007336  0.00248606  0.01686793 -0.00212772]
   [60] joint position: [ 0.07300469  0.00892482 -0.01490498  0.00350698  0.02387041 -0.00279907]
   [70] joint position: [ 0.098936    0.01293271 -0.02095646  0.00466479  0.03185446 -0.0034017 ]
   [80] joint position: [ 0.12865883  0.01808055 -0.02840175  0.00593683  0.04066865 -0.00385261]
   [90] joint position: [ 0.16206903  0.02458815 -0.03744247  0.00729732  0.05013223 -0.00406555]

You can find additional example code at https://github.com/dartsim/dart/tree/main/python/examples

Python logging
--------------

DART prints diagnostics to its native console sink by default. To send them
through Python's logging system, enable the bridge and drain its queue at a
Python boundary, such as after each simulation step:

.. code-block:: python

   import logging
   import dartpy as dart

   logging.basicConfig(level=logging.INFO)
   logger = logging.getLogger("dartpy")
   logger.setLevel(logging.WARNING)  # Python-side filter
   dart.common.enable_logging(level=logging.INFO)  # Native-side filter
   try:
       world = dart.simulation.World()
       world.setNumSimulationThreads(4)
       for _ in range(100):
           world.step()
           dart.common.flush_logs()
   finally:
       dart.common.disable_logging()

``enable_logging`` replaces the process-wide native sink; repeated calls update
its threshold without installing another bridge. Its keyword-only ``level``
uses Python logging constants. Values up to 10 (including ``NOTSET`` and
``DEBUG``) include native trace and debug; 11--20 select info, 21--30 warning,
31--40 error and 41--50 fatal. Values above 50 disable formatted log records.
Legacy C++ console streams bypass this native threshold. The
``dartpy`` logger's own level, filters, handlers and propagation apply at drain
time. Native trace and debug map to ``logging.DEBUG``, and fatal maps to
``logging.CRITICAL``.

Native threads only enqueue records. ``flush_logs()`` delivers one snapshot in
sink arrival order under Python's GIL, with no native logging or queue mutex
held while calling handlers. It returns the number dequeued, including records
filtered by Python. Thread scheduling determines the order between concurrent
producers; each producer's order is retained. The queue holds at most 100,000
records, including during work outside ``world.step()``. Overflow drops the
oldest records; the next flush emits one ``dartpy: N log records dropped``
warning through Python logging, separate from the returned dequeue count.
The dropped count resets before that warning is delivered, even if a handler
raises while processing the warning.
Drain regularly: no automatic flush occurs when a native call returns.

Handlers may log back into DART. Those records wait for the next drain;
calls made while another flush is running on the same thread return zero.
Other threads wait for that flush to finish, releasing the GIL while waiting.
Handlers must not join a thread that calls ``flush_logs()``: the handler and
that thread would wait for each other.
Handler exceptions propagate to the caller; the failed record is consumed and
remaining records stay queued.
``disable_logging()`` stops queueing, restores the previous sink if its wrapper
is still installed, restores the runtime threshold, then drains pending
records. If called from a handler, it returns zero and the outer flush drains the remaining queue, including handler-generated records.
The first enable registers ``disable_logging`` with ``atexit`` to deliver
records still queued at interpreter exit. The bridge and GUI App may shut down
in either order; inactive wrappers forward records to their previous sink.
When both shut down, retired wrappers are removed and the original sink is restored.
Nested GUI log captures must still be destroyed in reverse construction order
to restore stdout/stderr stream buffers correctly.

Default C++ ``dtmsg``, ``dtdbg``, ``dtwarn`` and ``dterr`` console output is
unchanged: text and legacy colors go immediately to stdout or stderr,
regardless of the native threshold. While a custom sink is installed, including
inside the GUI App, these streams assemble records, at info, debug, warning
and error severity
respectively. Their literal text, including percent signs, braces and legacy
prefixes, is preserved without added colors. Newline terminates a record;
``std::flush`` or ``std::endl``, the next ``dt*`` statement on that thread,
and thread exit deliver an unfinished line separately. Unterminated lines
inside the GUI App therefore also wait for one of those events, so their
terminal ordering relative to direct stdout/stderr writes can differ. Partial
lines produced before a sink swap use the sink installed when they are
flushed; swapping sinks does not flush them. Prefix-only statements (``dtmsg;``
or ``dtmsg << std::flush``) print the legacy prefix to the terminal under any
sink. A ``dt*`` call evaluated inside another ``dt*`` statement splits the
enclosing statement into separate records for custom sinks.
Recursive ``dt*`` output from a console-record callback goes
directly to stdout/stderr. The GUI log panel forwards native records to the
previous sink as well as displaying them, preserving legacy terminal bytes for
console records. On Windows, enabling virtual terminal processing for native
log colors leaves it enabled in that console.

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
