# DART Tutorials README

The purpose of this set of tutorials is to provide a quick introduction to
using DART. We designed many hands-on exercises to make the learning
effective and fun. To follow along with this tutorial, first locate
the tutorial code in the directory:
[dart/tutorials](https://github.com/dartsim/dart/blob/master/tutorials).
For each of the four tutorials, we provide the skeleton code as the starting
point (e.g. [tutorial_multi_pendulum.cpp]
(https://github.com/dartsim/dart/blob/master/tutorials/
tutorial_multi_pendulum.cpp)) and the final code as the answer to the tutorial
(e.g. [tutorial_multi_pendulum_finished.cpp](https://github.com/dartsim/dart/blob/master/tutorials/tutorial_multi_pendulum_finished.cpp)).

## Build Each Example

Copy the subdirectory to your workspace and follow the instruction of README.md
in the subdirectory.

## Build Examples as One Project

### Build Instructions

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

Copy this directory to your workspace (e.g., in Linux):

    $ cp -r tutorials /your/workspace/directory/dart_tutorials
    $ cd /your/workspace/directory/dart_tutorials

From the workspace directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

### Execute Instructions

Launch the each executable from the build directory above (e.g.,):

    $ ./tutorial_biped/tutorial_biped

Follow the instructions detailed in the console.

