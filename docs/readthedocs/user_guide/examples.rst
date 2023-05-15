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
