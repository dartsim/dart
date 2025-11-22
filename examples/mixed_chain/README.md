# Mixed Chain Example

This project is dependent on DART with OSG support. Please make sure a proper version of DART is installed with the gui component before building this project.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch the executable from the build directory above:

    $ ./mixed_chain

This example demonstrates simulation of articulated bodies consisting of both rigid bodies and soft bodies in a mixed chain configuration. The example loads a world containing articulated bodies with 10 bodies and allows interactive force application to soft body nodes.

Key features demonstrated:

- Mixed rigid-soft body chain simulation
- Interactive force application to soft body nodes
- Real-time visualization using OpenSceneGraph (OSG)
- Soft body dynamics with external forces

## Controls

- **q/w**: Apply force in -X/+X direction to the soft body
- **e/r**: Apply force in -Y/+Y direction to the soft body
- **t/y**: Apply force in -Z/+Z direction to the soft body
- **Space**: Toggle simulation on/off (standard OSG viewer control)

The simulation will start automatically. Use the keyboard controls listed above to interact with the soft body components of the articulated chain.
