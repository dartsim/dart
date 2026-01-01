# DART Examples README

## Overview

- Examples are being reorganized into numbered categories (pilot in progress).
- Start in `00_getting_started` for minimal, first-run examples.
- Continue with `01_rigid_bodies_and_frames` for basic rigid-body scenes.
- Explore `02_joints_and_constraints` for joint types and constraint behavior.
- Dig into `03_collisions_and_contacts` for collision and terrain workflows.
- Move on to `04_control_and_ik` for controllers and inverse kinematics.
- Visit `05_io_and_models` for model loading and resource retrieval.
- Check `06_soft_and_hybrid` for soft-body and mixed dynamics examples.
- Explore `07_visualization_and_interaction` for GUI and input patterns.
- See `08_performance_and_scaling` for timing and scaling examples.
- Tutorials remain in `tutorials/` and are cross-linked as categories expand.

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
