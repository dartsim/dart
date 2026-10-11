This example requires DART's Filament `gui` component, built with
`DART_BUILD_GUI=ON` (the default). CMake detects the installed
`dart/gui/Viewer.hpp` header independently of the reported DART version.
Configuration fails with a migration message if only the older GLUT GUI is
installed.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch the executable from the build directory above:

    $ ./{generated_executable}

Follow the instructions detailed in the console.

