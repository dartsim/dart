# Raylib Drag and Drop Example

Demonstrates drag-and-drop interaction with the DART Raylib viewer backend.

## Features

- **SimpleFrame Drag**: Left-click the green sphere marker to drag the red box
- **BodyNode IK Drag**: Left-click any part of the robot arm to drag with IK
- **Ctrl-Drag**: Rotate instead of translate

## Controls

- Right-drag: orbit camera
- Middle-drag: pan camera
- Scroll: zoom
- Left-click: select / start drag
- Space: pause/unpause
- Num5: toggle perspective/orthographic

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

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
