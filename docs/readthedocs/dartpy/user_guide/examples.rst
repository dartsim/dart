Examples
========

Once you have installed dartpy using pip install -U dartpy, you can run the following "Hello World" example to simulate a 6-DOF robot using DART.

.. note::

   In order to load the URDF, please clone dart repository and set the `DART_DATA_LOCAL_PATH` environment variable to where the `data` folder is in the cloned repository (e.g., `C:/ws/dart/data/` if cloned to `C:/ws/dart/`)

.. code-block:: python

    import dartpy as dart

    def main():
        world = dart.simulation.World()

        urdf_parser = dart.io.DartLoader()
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
   pip install "ssik>=2.3.0"

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

A complete, runnable demo -- which builds the DART skeleton from the solver's
product-of-exponentials chain, drives a draggable ``InteractiveFrame`` target in
an ``ImGuiViewer``, and re-solves the IK live as you drag -- is available at
``python/examples/ssik_analytical_ik/main.py``. It also ships a headless smoke
test over all ssik prebuilt arms:

.. code-block:: shell

   pixi run py-ex ssik_analytical_ik                                   # default arm (UR5)
   python python/examples/ssik_analytical_ik/main.py --arm franka_panda_ik
   python python/examples/ssik_analytical_ik/main.py --list           # list available arms
   python python/examples/ssik_analytical_ik/main.py --self-test      # headless check

A C++ counterpart, ``ssik_ik_gui`` (``pixi run ex ssik_ik_gui``), embeds a
Python interpreter to drive the same solver from an OSG/ImGui application.
