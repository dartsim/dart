# DART Tutorials README

The purpose of this set of tutorials is to provide a quick introduction to
using DART. We designed many hands-on exercises to make the learning
effective and fun. To follow along with this tutorial, first locate
the tutorial code in the directory: 
[dart/tutorials](https://github.com/dartsim/dart/blob/master/tutorials). 
For each of the four tutorials, we provide the skeleton code as the starting 
point (e.g. [tutorialMultiPendulum.cpp]
(https://github.com/dartsim/dart/blob/master/tutorials/
tutorialMultiPendulum.cpp)) and the final code as the answer to the tutorial
(e.g. [tutorialMultiPendulum-Finished.cpp](https://github.com/dartsim/dart/blob/master/tutorials/tutorialMultiPendulum-Finished.cpp)).

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

    $ ./tutorialBiped/tutorialBiped

Follow the instructions detailed in the console.

