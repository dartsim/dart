# DART Examples README

## Overview

- Examples live flat under `examples/` for simpler discovery and tooling.
- Category order is defined below; directory names stay short and stable.
- Tutorials remain in `tutorials/`.

## Categories (Ordered)

### 00 Getting Started

- `hello_world`
- `simple_frames`

### 01 Rigid Bodies and Frames

- `boxes`
- `hardcoded_design`
- `rigid_cubes`

### 02 Joints and Constraints

- `coupler_constraint`
- `free_joint_cases`
- `human_joint_limits`
- `joint_constraints`
- `mimic_pendulums`
- `rigid_chain`
- `rigid_loop`

### 03 Collisions and Contacts

- `box_stacking`
- `capsule_ground_contact`
- `heightmap`
- `lcp_physics`
- `rigid_shapes`

### 04 Control and IK

- `atlas_puppet`
- `atlas_simbicon`
- `biped_stand`
- `hubo_puppet`
- `operational_space_control`
- `vehicle`
- `wam_ikfast`

### 05 IO and Models

- `fetch`
- `g1_puppet`
- `unified_loading`

### 06 Soft and Hybrid

- `hybrid_dynamics`
- `mixed_chain`
- `soft_bodies`

### 07 Visualization and Interaction

- `add_delete_skels`
- `drag_and_drop`
- `empty`
- `imgui`
- `polyhedron_visual`
- `raylib`
- `simulation_event_handler`
- `tinkertoy`

### 08 Performance and Scaling

- `headless_simulation`
- `speed_test`

### 09 Integration and Tools

- `csv_logger`
- `point_cloud` (requires OctoMap)
- `rerun`

## Build Each Example

Copy the example directory to your workspace and follow the instruction of any
README.md in that example directory.

If you are working inside the DART repo, prefer the `pixi run` entry points
documented in `docs/onboarding/building.md` when available.

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
