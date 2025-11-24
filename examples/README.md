# DART Examples README

## Demo Browser

The consolidated ImGui-based browser lives in `examples/demo_app` (see its
README) if you want to browse multiple scenes from one window.

## Build Each Example

Copy the subdirectory to your workspace and follow the instruction of README.md
in the subdirectory.

## Build Examples as One Project

### Build Instructions

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

Copy this directory to your workspace (e.g., in Linux):

    $ cp -r examples /your/workspace/directory/dart_examples
    $ cd /your/workspace/directory/dart_examples

From the workspace directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

### Execute Instructions

Launch the each executable from the build directory above (e.g.,):

    $ ./hello_world

Follow the instructions detailed in the console.
